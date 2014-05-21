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
 * File Name   : arb_semaphore.c
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

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <string.h>
#include "arb_scheduler.h"
#include "arb_thread.h"
#include "arb_semaphore.h"
#include "utl_linkedlist.h"
#include "hal_pmic.h"
#include "hal_contextSwitch.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef struct t_sem
{
   t_semType t_type;

   int16_t s_count;

   t_CONTHNDL t_blockedList;

}t_semaphore;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static void arb_yieldFromWaitIntsOn( void) __attribute__\
 ( ( naked, noinline));
static void arb_yieldFromWaitIntsOff( void) __attribute__\
 ( ( naked, noinline));
static void arb_yieldFromSignalActiveInt( void) __attribute__\
 ( ( naked, noinline));
static void arb_yieldFromSignalIntsOn( void) __attribute__\
 ( ( naked, noinline));
static void arb_yieldFromSignalIntsOff( void) __attribute__\
 ( ( naked, noinline));

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
static t_semaphore *gpt_currentSem = NULL;
static t_tcb *gpt_temp;
static t_linkedListError gt_linkError;

/*---------------------------------------------------------------------------*
 * Linked list of all the open semaphores running on the system.
 *---------------------------------------------------------------------------*/
UTL_CREATE_CONTAINER( gt_activeSems);

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * This function performs the same operations as 'arb_yield' except that the
 * context save routine saves the contents of the SREG with the GIE bit set.
 *---------------------------------------------------------------------------*/
static void arb_yieldFromSignalActiveInt( void)
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
    * thread was running at the time of preemption will be executed. Since
    * this function can be indirectly called from within an interrupt routine,
    * we need to make a "reti" call so that the PMIC controller will be
    * returned to the state it had before entering the interrupt - which means
    * clearing the '.... Interrupt Executing flag'. Otherwise interrupts
    * won't be enabled until the calling thread is given back the processor
    * by the scheduler and the initial interrupt that indirectly made this
    * yield call is allowed to finish.
    *------------------------------------------------------------------------*/
	HAL_RETI();

}/*End arb_yieldFromSignalActiveInt*/

static void arb_yieldFromSignalIntsOn( void)
{

   /*------------------------------------------------------------------------*
    * Save the current thread's stack and CPU registers.
    *------------------------------------------------------------------------*/
   hal_contextSaveWithIntsOn(); /*SREG is saved with interrupts enabled*/

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

}/*End arb_yieldFromSignalIntsOn*/

static void arb_yieldFromSignalIntsOff( void)
{

   /*------------------------------------------------------------------------*
    * Save the current thread's stack and CPU registers.
    *------------------------------------------------------------------------*/
   hal_contextSaveWithIntsOff(); /*SREG is saved with interrupts enabled*/

   /*------------------------------------------------------------------------*
    * Now switch over to the kernel's stack in order to leave the previous
    * thread's stack unchanged while running the kernel thread scheduler.
    *------------------------------------------------------------------------*/
   hal_switchToKernelStack();

   /*------------------------------------------------------------------------*
    * Check to see if this thread overflowed its stack by checking the
    * secret number that was inserted at the end of the stack.
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

}/*End arb_yieldFromSignalIntsOff*/

static void arb_yieldFromWaitIntsOn( void)
{

   /*------------------------------------------------------------------------*
    * Save the current thread's stack and CPU registers.
    *------------------------------------------------------------------------*/
   hal_contextSaveWithIntsOn(); /*SREG is saved with interrupts enabled*/

   /*------------------------------------------------------------------------*
    * Now switch over to the kernel's stack in order to leave the previous
    * thread's stack unchanged while running the kernel thread scheduler.
    *------------------------------------------------------------------------*/
   hal_switchToKernelStack();

   /*------------------------------------------------------------------------*
    * Check to see if this thread overflowed its stack by checking the
    * secret number that was inserted at the end of the stack.
    *------------------------------------------------------------------------*/
   gpt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR_CONT_CURR(gt_activeThreads);
   if( gpt_temp->ps_stack[0] != ARB_STACK_OVERFLOW_CHECK)
   {
      arb_stackOverflow();
   }

   /*------------------------------------------------------------------------*
    * Change the status to BLOCKED.
    *------------------------------------------------------------------------*/
   gpt_temp->t_status = BLOCKED;

   /*------------------------------------------------------------------------*
    * Remove the currently running thread from the active list.
    *------------------------------------------------------------------------*/
   gt_linkError = utl_removeLink( gt_activeThreads,
                                  UTL_GET_CURR_OF_CONT( gt_activeThreads));

   /*------------------------------------------------------------------------*
    * Insert this thread onto the blocked list for this semaphore...
    *------------------------------------------------------------------------*/
   gt_linkError = utl_insertLink( gpt_currentSem->t_blockedList,
                                  UTL_GET_CURR_OF_CONT( gt_activeThreads),
                                  true);

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
    * thread was running at the time of preemption will be executed. Since
    * this function can be indirectly called from within an interrupt routine,
    * we need to make a "reti" call so that the PMIC controller will be
    * returned to the state it had before entering the interrupt - which means
    * clearing the '.... Interrupt Executing flag'. Otherwise interrupts
    * won't be enabled until the calling thread is given back the processor
    * by the scheduler and the initial interrupt that indirectly made this
    * yield call is allowed to finish.
    *
    * Calling this function outside and interrupt routine doesn't seem to
    * cause any issues - but I am not completely certain. I may need to make
    * an additional 'yield' routing that is called outside and interrupt and
    * make a decision on which one to run based on the status of the PMIC
    * controller.
    *------------------------------------------------------------------------*/
	HAL_RET();

}/*End arb_yieldFromWaitIntsOn*/

static void arb_yieldFromWaitIntsOff( void)
{

   /*------------------------------------------------------------------------*
    * Save the current thread's stack and CPU registers.
    *------------------------------------------------------------------------*/
   hal_contextSaveWithIntsOff();/*SREG is saved with interrupts disabled*/

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
    * Change the status to BLOCKED.
    *------------------------------------------------------------------------*/
   gpt_temp->t_status = BLOCKED;

   /*------------------------------------------------------------------------*
    * Remove the currently running thread from the active list.
    *------------------------------------------------------------------------*/
   gt_linkError = utl_removeLink( gt_activeThreads,
                                  UTL_GET_CURR_OF_CONT( gt_activeThreads));

   /*------------------------------------------------------------------------*
    * Insert this thread onto the blocked list for this semaphore...
    *------------------------------------------------------------------------*/
   gt_linkError = utl_insertLink( gpt_currentSem->t_blockedList,
                                  UTL_GET_CURR_OF_CONT( gt_activeThreads),
                                  true);

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
    * thread was running at the time of preemption will be executed. Since
    * this function can be indirectly called from within an interrupt routine,
    * we need to make a "reti" call so that the PMIC controller will be
    * returned to the state it had before entering the interrupt - which means
    * clearing the '.... Interrupt Executing flag'. Otherwise interrupts
    * won't be enabled until the calling thread is given back the processor
    * by the scheduler and the initial interrupt that indirectly made this
    * yield call is allowed to finish.
    *
    * Calling this function outside and interrupt routine doesn't seem to
    * cause any issues - but I am not completely certain. I may need to make
    * an additional 'yield' routing that is called outside and interrupt and
    * make a decision on which one to run based on the status of the PMIC
    * controller.
    *------------------------------------------------------------------------*/
	HAL_RET();

}/*End arb_yieldFromWaitIntsOff*/

t_error arb_wait( t_SEMHANDLE t_semHandle,
                  t_semMode t_mode)
{
  t_tcb *pt_temp;

   /*------------------------------------------------------------------------*
    * Since we are passing in an argument to this function we cannot use
    * the 'naked' attribute - if we were to, the contents of the calling
    * threads stack will be overwritten do to improper handling of the stack
    * frame pointer (or 'Y' register)- basically we haven't added room on the
    * stack for the function parameters. Therefore, by calling this function
    * 'normally' the compiler will add its own prologue and epilogue routines
    * (remembering the program counter, stack variables, and CPU registers of
    * the calling thread). Any use of the stack from within this function will
    * be stored on the stack of the calling thread and care needs to be
    * taken to make sure it size is large enough to handle the extra workload.
    *------------------------------------------------------------------------*/
   if( HAL_ARE_INTS_EN()) /*Interrupt enabled?*/
   {

      /*---------------------------------------------------------------------*
       * Perform mutual exclusion.
       *---------------------------------------------------------------------*/
      HAL_CLI();

      if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_semHandle, gt_activeSems) ==
      false)
      {
         HAL_SEI();
         return ARB_INVALID_HANDLE;
      }

      gpt_currentSem = (t_semaphore *)UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)\
      t_semHandle);

      if( t_mode == BLOCKING)
      {
         /*------------------------------------------------------------------*
          * Decrement the waiting count.
          *------------------------------------------------------------------*/
         gpt_currentSem->s_count--;

         /*------------------------------------------------------------------*
          * Is it time for this thread to be blocked?
          *------------------------------------------------------------------*/
         if( gpt_currentSem->s_count < 0) /*Yes, block process*/
         {
            /*---------------------------------------------------------------*
             * Record the maximum time this thread will wait on this semaphore
             *---------------------------------------------------------------*/
            pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR_CONT_CURR\
            (gt_activeThreads);

            //pt_temp->s_quantum = s_quantum;

            /*---------------------------------------------------------------*
             * Since we previously disabled interrupts this yield call saves
             * the contents of the SREG with the GIE bit enabled. It assumes
             * that the thread being restored with the context save has
             * interrupts already enabled.
             *---------------------------------------------------------------*/
            arb_yieldFromWaitIntsOn();

            /*---------------------------------------------------------------*
             * If you have returned back to this point you are operating on
             * the stack of the thread that initially made the call into 'wait'
             *---------------------------------------------------------------*/

         }/*End if( pt_semHandle->s_count < 0)*/
         else /*No, continue process*/
            HAL_SEI(); /*Enable interrupts*/
      }
      else /*Non blocking*/
      {
         if( gpt_currentSem->s_count > 0)
         {
            gpt_currentSem->s_count--;
            HAL_SEI(); /*Enable interrupts*/
         }
         else
         {
            HAL_SEI(); /*Enable interrupts*/
            return ARB_SEM_DEC_ERROR; /*Failed to decrement semaphore*/
         }
      }

   }/*End if( HAL_ARE_INTS_EN())*/
   else
   {

      if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_semHandle, gt_activeSems) ==
      false)
      {
         return ARB_INVALID_HANDLE;
      }

      gpt_currentSem = (t_semaphore *)UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)\
      t_semHandle);

      if( t_mode == BLOCKING)
      {

         /*------------------------------------------------------------------*
          * Decrement the waiting count.
          *------------------------------------------------------------------*/
         gpt_currentSem->s_count--;

         /*------------------------------------------------------------------*
          * Is it time for this thread to be blocked?
          *------------------------------------------------------------------*/
         if( gpt_currentSem->s_count < 0) /*Yes, block process*/
         {
            /*---------------------------------------------------------------*
             * Record the maximum time this thread will wait on this semaphore
             *---------------------------------------------------------------*/
            pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR_CONT_CURR\
            (gt_activeThreads);

            //pt_temp->s_quantum = s_quantum;

            /*---------------------------------------------------------------*
             * Since we previously disabled interrupts this yield call saves
             * the contents of the SREG with the GIE bit enabled.
             *---------------------------------------------------------------*/
            arb_yieldFromWaitIntsOff();

            /*---------------------------------------------------------------*
             * If you have returned back to this point you are operating on
             * the stack of the thread that initially made the call into 'wait'
             *---------------------------------------------------------------*/

         }/*End if( pt_semHandle->s_count < 0)*/

      }
      else /*Non blocking*/
      {
         if( gpt_currentSem->s_count > 0)
         {
            gpt_currentSem->s_count--;
         }
         else
         {
            return ARB_SEM_DEC_ERROR; /*Failed to decrement semaphore*/
         }
      }

   }/*Interrupts disabled*/

   /*------------------------------------------------------------------------*
    * Since the call into 'wait' wasn't 'naked' we will let the compiler
    * return us to the calling thread's next program counter location.
    *------------------------------------------------------------------------*/
   return ARB_PASSED;

}/*End arb_wait( )*/

t_error arb_signal( t_SEMHANDLE t_semHandle)
{
   uint16_t s_count;
   t_LINKHNDL t_curr;
   t_LINKHNDL t_highest;
   t_LINKHNDL t_prevLink;
   uint16_t s_highest;
   t_tcb *pt_temp;
   t_tcb *pt_temp2;
   t_linkedListError t_linkError;

   /*------------------------------------------------------------------------*
    * Since we are passing in an argument to this function we cannot use
    * the 'naked' attribute - if we were to, the contents of the calling
    * threads stack will be overwritten do to improper handling of the stack
    * frame pointer (or 'Y' register)- basically we haven't added room on the
    * stack for the function parameters. Therefore, by calling this function
    * 'normally' the compiler will add its own prologue and epilogue routines
    * (remembering the program counter, stack variables, and CPU registers of
    * the calling thread). Any use of the stack from within this function will
    * be stored on the stack of the calling thread and care needs to be
    * taken to make sure it size is larger enough to handle the extra workload.
    *------------------------------------------------------------------------*/
   if( HAL_ARE_INTS_EN()) /*Interrupts enabled?*/
   {
      /*---------------------------------------------------------------------*
       * Mutual exclusion
       *---------------------------------------------------------------------*/
      HAL_CLI();

      /*---------------------------------------------------------------------*
       * Is this semaphore valid?
       *---------------------------------------------------------------------*/
      if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_semHandle, gt_activeSems) ==
      false)
      {
         HAL_SEI();
         return ARB_INVALID_HANDLE;
      }

      gpt_currentSem = (t_semaphore *)UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)\
      t_semHandle);

      /*---------------------------------------------------------------------*
       * Remove a thread from the waiting count.
       *---------------------------------------------------------------------*/
      gpt_currentSem->s_count++;

      /*---------------------------------------------------------------------*
       * Are there threads waiting on this semaphore, if so then wake
       * according to priority.
       *---------------------------------------------------------------------*/
      if( (gpt_currentSem->s_count <= 0) &&
      (UTL_GET_NUM_LINKS_CONT( gpt_currentSem->t_blockedList) > 0))
      {

         /*------------------------------------------------------------------*
          * Find the thread with the highest priority on the blocked
          * list for this semaphore.
          *------------------------------------------------------------------*/
         t_highest = UTL_GET_HEAD_OF_CONT( gpt_currentSem->t_blockedList);
         pt_temp   = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_highest);
         s_highest = pt_temp->t_priority;
         UTL_TRAVERSE_CONTAINER_HEAD( t_curr, gpt_currentSem->t_blockedList,\
         s_count)
         {
            pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_curr);
            if( pt_temp->t_priority < s_highest)
            {
               s_highest = pt_temp->t_priority;
               t_highest = t_curr;
            }
         }

         /*------------------------------------------------------------------*
          * Only move the highest priority thread off the blocked list and
          * insert back on the active list.
          *------------------------------------------------------------------*/
         if( gpt_currentSem->t_type != SIGNAL) /*Remove highest...*/
         {
            /*---------------------------------------------------------------*
             * Change the status of the highest priority thread on the
             * blocked list to READY.
             *---------------------------------------------------------------*/
            pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_highest);
            pt_temp->t_status = READY;

            /*---------------------------------------------------------------*
             * Remove the highest priority thread from the blocked list
             *---------------------------------------------------------------*/
            t_linkError = utl_removeLink( gpt_currentSem->t_blockedList,
                                          t_highest);

            /*---------------------------------------------------------------*
             * Insert this thread onto the active list.
             *---------------------------------------------------------------*/
            t_linkError = utl_insertLink( gt_activeThreads,
                                          t_highest,
                                          true);

         }/*End if( gpt_currentSem->t_type != SIGNAL)*/
         else /*SIGNAL...remove all*/
         {

            /*----------------------------------------------------------------*
             * Remove all the threads waiting on the blocked list and insert
             * them onto the active list. Since UTL_TRAVERSE_CONTAINER
             * traverses through the entire list using the current position of
             * t_curr once its deleted the loop will not be unable to move to
             * the next ptr location. This issue is resolved by using a
             * previous ptr to change t_curr back to a valid location once an
             * item has been removed.
             *----------------------------------------------------------------*/
            UTL_TRAVERSE_CONTAINER_HEAD( t_curr, gpt_currentSem->t_blockedList,\
            s_count)
            {
               pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_curr);

               pt_temp->t_status = READY;

               t_prevLink = UTL_GET_PREV_LINK( t_curr);

               t_linkError = utl_removeLink( gpt_currentSem->t_blockedList,
                                             t_curr);

               t_linkError = utl_insertLink( gt_activeThreads,
                                             t_curr,
                                             true);
               t_curr = t_prevLink;
            }

            /*---------------------------------------------------------------*
             * Since we removed all the threads from the blocked list, set
             * the count back to 0.
             *---------------------------------------------------------------*/
            gpt_currentSem->s_count = 0;

         }

         /*------------------------------------------------------------------*
          * If the highest priority thread we removed from the BLOCKED list
          * has a higher priority then the current running thread then
          * suspend the current thread.
          *------------------------------------------------------------------*/
         pt_temp  = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_highest);
         pt_temp2 = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR_CONT_CURR\
         (gt_activeThreads);
         if( pt_temp->t_priority <= pt_temp2->t_priority)
         {

            /*---------------------------------------------------------------*
             * Change the status of the current thread from active to READY.
             *---------------------------------------------------------------*/
            pt_temp2->t_status = READY;

            /*---------------------------------------------------------------*
             * The thread we removed from the BLOCKED list and put back on
             * the active list has a higher priority then the signaling
             * thread. Calling yield will cause the signaling thread to
             * give up its time slice and allow the next higher priority
             * thread (which should be the one we just removed from the
             * BLOCKED list) to run.
             *---------------------------------------------------------------*/
            if( HAL_IS_ACTIVE_INT() > 0)
            {
               /*------------------------------------------------------------*
                * This routine was called from an active interrupt.
                * Therefore, we are calling a special yield function that
                * re enables interrupts when it switches in a new thread.
                *------------------------------------------------------------*/
               arb_yieldFromSignalActiveInt();
            }/*End if( HAL_IS_ACTIVE_INT > 0)*/
            else
            {
               arb_yieldFromSignalIntsOn();
            }

            /*---------------------------------------------------------------*
             * If you have returned back to this point you are operating on
             * the stack of the thread that initially made the call into
             * 'signal' and returning from this non 'naked' function call
             * will return you to your previous set of operations.
             *---------------------------------------------------------------*/

         }
         else
         {
            HAL_SEI();
         }

      }/*End if( gpt_currentSem->s_count <= 0)*/
      else
      {
         HAL_SEI();
      }

   }/*End if( HAL_ARE_INTS_EN())*/
   else
   {

      if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_semHandle, gt_activeSems) ==
      false)
      {
         return ARB_INVALID_HANDLE;
      }

      gpt_currentSem = (t_semaphore *)UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)\
      t_semHandle);

      /*---------------------------------------------------------------------*
       * Remove a thread from the waiting count.
       *---------------------------------------------------------------------*/
      gpt_currentSem->s_count++;

      /*---------------------------------------------------------------------*
       * Are there threads waiting on this semaphore, if so then wake
       * according to priority.
       *---------------------------------------------------------------------*/
      if( (gpt_currentSem->s_count <= 0) &&
      (UTL_GET_NUM_LINKS_CONT( gpt_currentSem->t_blockedList) > 0))
      {
         /*------------------------------------------------------------------*
          * Find the thread with the highest priority on the blocked
          * list for this semaphore.
          *------------------------------------------------------------------*/
         t_highest = UTL_GET_HEAD_OF_CONT( gpt_currentSem->t_blockedList);
         pt_temp   = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_highest);
         s_highest = pt_temp->t_priority;
         UTL_TRAVERSE_CONTAINER_HEAD( t_curr, gpt_currentSem->t_blockedList,\
         s_count)
         {
            pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_curr);
            if( pt_temp->t_priority < s_highest)
            {
               s_highest = pt_temp->t_priority;
               t_highest = t_curr;
            }
         }

         /*------------------------------------------------------------------*
          * Only move the highest priority thread off the blocked list and
          * insert back on the active list.
          *------------------------------------------------------------------*/
         if( gpt_currentSem->t_type != SIGNAL)
         {
            /*---------------------------------------------------------------*
             * Change the status of the highest priority thread on the
             * blocked list to READY.
             *---------------------------------------------------------------*/
            pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_highest);
            pt_temp->t_status = READY;

            /*---------------------------------------------------------------*
             * Remove the highest priority thread from the blocked list
             *---------------------------------------------------------------*/
            t_linkError = utl_removeLink( gpt_currentSem->t_blockedList,
                                          t_highest);

            /*---------------------------------------------------------------*
             * Insert this thread onto the active list.
             *---------------------------------------------------------------*/
            t_linkError = utl_insertLink( gt_activeThreads,
                                          t_highest,
                                          true);

         }/*End if( gpt_currentSem->t_type != SIGNAL)*/
         else /*SIGNAL*/
         {

            /*----------------------------------------------------------------*
             * Remove all the threads waiting on the blocked list and insert
             * them onto the active list. Since UTL_TRAVERSE_CONTAINER
             * traverses through the entire list using the current position of
             * t_curr once its deleted the loop will not be unable to move to
             * the next ptr location. This issue is resolved by using a
             * previous ptr to change t_curr back to a valid location once an
             * item has been removed.
             *----------------------------------------------------------------*/
            UTL_TRAVERSE_CONTAINER_HEAD( t_curr, gpt_currentSem->t_blockedList,\
            s_count)
            {
               pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_curr);

               pt_temp->t_status = READY;

               t_prevLink = UTL_GET_PREV_LINK( t_curr);

               t_linkError = utl_removeLink( gpt_currentSem->t_blockedList,
                                             t_curr);

               t_linkError = utl_insertLink( gt_activeThreads,
                                             t_curr,
                                             true);
               t_curr = t_prevLink;
            }

            /*---------------------------------------------------------------*
             * Since we removed all the threads from the blocked list, set
             * the count back to 0.
             *---------------------------------------------------------------*/
            gpt_currentSem->s_count = 0;

         }

         /*------------------------------------------------------------------*
          * If the highest priority thread we removed from the BLOCKED list
          * has a higher priority then the current running thread then
          * suspend the current thread.
          *------------------------------------------------------------------*/
         pt_temp  = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_highest);
         pt_temp2 = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR_CONT_CURR\
         (gt_activeThreads);
         if( pt_temp->t_priority <= pt_temp2->t_priority)
         {

            /*---------------------------------------------------------------*
             * Change the status of the current thread from active to READY.
             *---------------------------------------------------------------*/
            pt_temp2->t_status = READY;

            /*---------------------------------------------------------------*
             * The thread we removed from the BLOCKED list and put back on
             * the active list has a higher priority then the signaling
             * thread. Calling yield will cause the signaling thread to
             * give up its time slice and allow the next higher priority
             * thread (which should be the one we just removed from the
             * BLOCKED list) to run.
             *---------------------------------------------------------------*/
            arb_yieldFromSignalIntsOff();

            /*---------------------------------------------------------------*
             * If you have returned back to this point you are operating on
             * the stack of the thread that initially made the call into
             * 'signal' and returning from this non 'naked' function call
             * will return you to your previous set of operations.
             *---------------------------------------------------------------*/

         }

      }/*End if( gpt_currentSem->s_count <= 0)*/

   }

   /*------------------------------------------------------------------------*
    * Since the call into 'signal' wasn't 'naked' we will let the compiler
    * return us to the calling thread's next program counter location.
    *------------------------------------------------------------------------*/
   return ARB_PASSED;

}/*End arb_signal( t_SEMHANDLE *pt_semHandle)*/

t_SEMHANDLE arb_semaphoreCreate( t_semType t_type)
{
   t_semaphore *pt_newSem;
   t_LINKHNDL t_linkHndl;
   t_linkedListError t_err;

   /*------------------------------------------------------------------------*
    * We are going to be adding an element to a shared list so enforce
    * mutual exclusion.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   /*------------------------------------------------------------------------*
    * Create a new link handle that will store information about this
    * semaphore.
    *------------------------------------------------------------------------*/
   t_linkHndl = utl_createLink( sizeof(t_semaphore));

   if( t_linkHndl < 0)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return (t_SEMHANDLE)ARB_OUT_OF_HEAP;
   }/*End if( t_linkHndl < 0)*/

   /*------------------------------------------------------------------------*
    * Add the semaphore 'link' onto the list containing all the open
    * semaphores on the system.
    *------------------------------------------------------------------------*/
   t_err = utl_insertLink( gt_activeSems,
                           t_linkHndl,
                           true);

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where information
    * about the semaphore is stored.
    *------------------------------------------------------------------------*/
   pt_newSem = (t_semaphore *)UTL_GET_LINK_ELEMENT_PTR( t_linkHndl);

   /*------------------------------------------------------------------------*
    * Each semaphore contains a blocked list where threads waiting on the
    * semaphore will be stored.
    *------------------------------------------------------------------------*/
   pt_newSem->t_blockedList = utl_createContainer();
   if( pt_newSem->t_blockedList < 0)
   {
      t_err = utl_destroyLink( gt_activeSems,
                               t_linkHndl);
      HAL_END_CRITICAL();//Enable interrupts
      return (t_SEMHANDLE)ARB_OUT_OF_HEAP;
   }

   /*------------------------------------------------------------------------*
    * Are we a SIGNAL or COUNTING semaphore?
    *------------------------------------------------------------------------*/
   pt_newSem->t_type = t_type;

   if( t_type == MUTEX)
   {
      /*---------------------------------------------------------------------*
       * This semaphore is used for mutual exclusion initialize to 1 or
       * 'unlocked'.
       *---------------------------------------------------------------------*/
      pt_newSem->s_count = 1;
   }
   else
      pt_newSem->s_count = 0;

   HAL_END_CRITICAL();//Enable interrupts

   /*------------------------------------------------------------------------*
    * Return a handle (or address ) to this semaphore.
    *------------------------------------------------------------------------*/
   return (t_SEMHANDLE)t_linkHndl;

}/*End arb_semaphoreCreate*/

t_error arb_semaphoreDestroy( t_SEMHANDLE t_semHandle)
{
   t_semaphore *pt_sem = NULL;
   uint16_t s_count;
   t_LINKHNDL t_curr;
   t_LINKHNDL t_prevLink;
   t_linkedListError t_err;
   t_tcb *pt_temp;

   /*------------------------------------------------------------------------*
    * We are going to be removing a global element so perform mutual
    * exclusion.
    *------------------------------------------------------------------------*/
   HAL_BEGIN_CRITICAL(); //Disable interrupts

   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_semHandle, gt_activeSems) ==
   false)
   {
      HAL_END_CRITICAL();//Enable interrupts
      return ARB_INVALID_HANDLE;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where information
    * about the semaphore is stored.
    *------------------------------------------------------------------------*/
   pt_sem = (t_semaphore *)UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)\
   t_semHandle);

   /*------------------------------------------------------------------------*
    * Remove all the threads waiting on the blocked list and insert them onto
    * the active list. Since UTL_TRAVERSE_CONTAINER traverses through the
    * entire list using the current position of t_curr once its deleted the
    * loop will not be unable to move to the next ptr location. This issue is
    * resolved by using a previous ptr to change t_curr back to a valid
    * location once an item has been removed.
    *------------------------------------------------------------------------*/
   UTL_TRAVERSE_CONTAINER_HEAD( t_curr, pt_sem->t_blockedList, s_count)
   {
      pt_temp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR( t_curr);

      pt_temp->t_status = READY;

      t_prevLink = UTL_GET_PREV_LINK( t_curr);

      t_err = utl_removeLink( pt_sem->t_blockedList,
                              t_curr);

      t_err = utl_insertLink( gt_activeThreads,
                              t_curr,
                              true);
      t_curr = t_prevLink;
   }

   /*------------------------------------------------------------------------*
    * Destroy the blocked list for this semaphore.
    *------------------------------------------------------------------------*/
   t_err = utl_destroyContainer( pt_sem->t_blockedList);

   /*------------------------------------------------------------------------*
    * Destroy the semaphore
    *------------------------------------------------------------------------*/
   t_err = utl_destroyLink( gt_activeSems, (t_LINKHNDL)t_semHandle);

   HAL_END_CRITICAL();//Enable interrupts

   return ARB_PASSED;

}/*End arb_semaphoreDestroy*/

t_error arb_semaphoreInit( t_SEMHANDLE t_semHandle,/*Pointer to the semaphores handle*/
                           int16_t     s_value)
{
   t_semaphore *pt_sem = NULL;

   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_semHandle, gt_activeSems) == false)
   {
      return ARB_INVALID_HANDLE;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where information
    * about the semaphore is stored.
    *------------------------------------------------------------------------*/
   pt_sem = (t_semaphore *)UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)\
   t_semHandle);

   pt_sem->s_count = s_value;

   return ARB_PASSED;

}/*End arb_semaphoreInit*/

int16_t arb_semaphoreGetCount( t_SEMHANDLE t_semHandle)
{
   t_semaphore *pt_sem = NULL;

   if( UTL_IS_LINK_ON_LIST( (t_LINKHNDL)t_semHandle, gt_activeSems) == false)
   {
      return ARB_INVALID_HANDLE;
   }

   /*------------------------------------------------------------------------*
    * Get a ptr to the link's element- which is the area where information
    * about the semaphore is stored.
    *------------------------------------------------------------------------*/
   pt_sem = (t_semaphore *)UTL_GET_LINK_ELEMENT_PTR( (t_LINKHNDL)\
   t_semHandle);

   return pt_sem->s_count;

}/*End arb_semaphoreGetCount*/

