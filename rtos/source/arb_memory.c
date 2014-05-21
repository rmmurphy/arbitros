/*---------------------------------------------------------------------------*
 * Copyright (C) 2011-2012 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * File Name   : arb_memory.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for the dynamic creation/destruction
 *               of memory from within or outside of thread space.
 *
 * Last Update : June, 28, 2013
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include "arb_sysTimer.h"
#include "arb_scheduler.h"
#include "hal_pmic.h"
#include "hal_contextSwitch.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static void arb_createMemIntsOn( void) __attribute__ ( ( naked, noinline));
static void arb_createMemIntsOff( void) __attribute__ ( ( naked, noinline));

/*---------------------------------------------------------------------------*
 * Private Typedefs
 *---------------------------------------------------------------------------*/
typedef struct
{
   /*------------------------------------------------------------------------*
    * Size of the memory region that is going to be created.
    *------------------------------------------------------------------------*/
   size_t gs_sizeMem;

   /*------------------------------------------------------------------------*
    * Pointer to the pointer containing the created memory region.
    *------------------------------------------------------------------------*/
   void **gpv_newMem;

   /*------------------------------------------------------------------------*
    * If true, a malloc operation is performed, otherwise free.
    *------------------------------------------------------------------------*/
   bool b_performMalloc;

   /*------------------------------------------------------------------------*
    * Temp ptr to the calling thread.
    *------------------------------------------------------------------------*/
   t_tcb *pt_temp;

}t_memObject;

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
static t_memObject gt_memObject;

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
static void arb_createMemIntsOff( void)
{

   /*------------------------------------------------------------------------*
    * Save the current thread's stack and CPU registers.
    *------------------------------------------------------------------------*/
   hal_contextSaveWithIntsOff(); /*SREG is saved with interrupts enabled*/

   /*------------------------------------------------------------------------*
    * Now switch over to the kernel's stack in order to leave the previous
    * thread's stack unchanged while running the kernel thread scheduler. The
    * Kernel stack was not created using malloc- which allows for the dynamic
    * creation of memory during active os operation. Otherwise, dynamic
    * creation of memory is only possible before the system timer is started.
    *------------------------------------------------------------------------*/
   hal_switchToKernelStack();

   /*------------------------------------------------------------------------*
    * Check to see if this thread overflowed its stack by checking the
    * secret number that was inserted at the end of the stack.
    *------------------------------------------------------------------------*/
   gt_memObject.pt_temp =
   (t_tcb *)UTL_GET_LINK_ELEMENT_PTR_CONT_CURR(gt_activeThreads);
   if( (gt_memObject.pt_temp->ps_stack[0] != ARB_STACK_OVERFLOW_CHECK)
       && (gt_memObject.pt_temp != NULL))
   {
      arb_stackOverflow();
   }

   if( gt_memObject.b_performMalloc == true)
   {
      *(gt_memObject.gpv_newMem) = malloc( gt_memObject.gs_sizeMem);
   }
   else
      free( *gt_memObject.gpv_newMem);

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

}/*End arb_createMemIntsOff*/

static void arb_createMemIntsOn( void)
{

   /*------------------------------------------------------------------------*
    * Save the current thread's stack and CPU registers.
    *------------------------------------------------------------------------*/
   hal_contextSaveWithIntsOn(); /*SREG is saved with interrupts enabled*/

   /*------------------------------------------------------------------------*
    * Now switch over to the kernel's stack in order to leave the previous
    * thread's stack unchanged while running the kernel thread scheduler. The
    * Kernel stack was not created using malloc- which allows for the dynamic
    * creation of memory during active os operation. Otherwise, dynamic
    * creation of memory is only possible before the system timer is started.
    *------------------------------------------------------------------------*/
   hal_switchToKernelStack();

   /*------------------------------------------------------------------------*
    * Check to see if this thread overflowed its stack by checking the
    * secret number that was inserted at the end of the stack.
    *------------------------------------------------------------------------*/
   gt_memObject.pt_temp =
   (t_tcb *)UTL_GET_LINK_ELEMENT_PTR_CONT_CURR(gt_activeThreads);
   if( (gt_memObject.pt_temp->ps_stack[0] !=
       ARB_STACK_OVERFLOW_CHECK) && (gt_memObject.pt_temp != NULL))
   {
      arb_stackOverflow();
   }

   if( gt_memObject.b_performMalloc == true)
   {
      *(gt_memObject.gpv_newMem) = malloc( gt_memObject.gs_sizeMem);
   }
   else
      free( *(gt_memObject.gpv_newMem));

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

}/*End arb_createMemIntsOn*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
void arb_malloc( size_t t_size,
                 void **pv_newMem)
{
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
   if( arb_sysTimerEnabled() == true)
   {
      /*---------------------------------------------------------------------*
       * WARNING - this assumes we are running within thread space!
       * Threads are active - use context switching in order to save the state
       * of the system while allocating a new memory region.
       *---------------------------------------------------------------------*/
      if( HAL_ARE_INTS_EN()) /*Interrupt enabled?*/
      {
         /*------------------------------------------------------------------*
          * Perform mutual exclusion.
          *------------------------------------------------------------------*/
         HAL_CLI();
         gt_memObject.b_performMalloc = true;
         gt_memObject.gpv_newMem = pv_newMem;
         gt_memObject.gs_sizeMem = t_size;
         arb_createMemIntsOn();

      }/*End if( HAL_ARE_INTS_EN())*/
      else
      {
         gt_memObject.b_performMalloc = true;
         gt_memObject.gpv_newMem = pv_newMem;
         gt_memObject.gs_sizeMem = t_size;
         arb_createMemIntsOff();
      }

   }/*End if( arb_sysTimerEnabled() == true)*/
   else
   {
      /*---------------------------------------------------------------------*
       * WARNING - this assumes we are not running within thread space!
       * Threads are not active, no context switching required.
       *---------------------------------------------------------------------*/
      (*pv_newMem) = malloc( t_size);
   }

}/*End arb_malloc*/

void arb_free( void **pv_newMem)
{
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
   if( arb_sysTimerEnabled() == true)
   {
      /*---------------------------------------------------------------------*
       * WARNING - this assumes we are running within thread space!
       * Threads are active - use context switching in order to save the state
       * of the system while allocating a new memory region.
       *---------------------------------------------------------------------*/
      if( HAL_ARE_INTS_EN()) /*Interrupt enabled?*/
      {
         /*------------------------------------------------------------------*
          * Perform mutual exclusion.
          *------------------------------------------------------------------*/
         HAL_CLI();
         gt_memObject.b_performMalloc = false;
         gt_memObject.gpv_newMem = pv_newMem;
         arb_createMemIntsOn();

      }/*End if( HAL_ARE_INTS_EN())*/
      else
      {
         gt_memObject.b_performMalloc = false;
         gt_memObject.gpv_newMem = pv_newMem;
         arb_createMemIntsOff();
      }

   }/*End if( arb_sysTimerEnabled() == true)*/
   else
   {
      /*---------------------------------------------------------------------*
       * WARNING - this assumes we are not running within thread space!
       * Threads are not active, no context switching required.
       *---------------------------------------------------------------------*/
      free( (*pv_newMem));
   }

}/*End arb_free*/