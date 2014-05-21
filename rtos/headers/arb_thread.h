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
 * File Name   : arb_thread.h
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
#ifndef arb_thread_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define arb_thread_h
   #define RUN_FOREVER (1)

   /*------------------------------------------------------------------------*
    * Use this variable for declaring 'extra' stack space for storing
    * CPU specific registers beyond that which is part of the user space
    * application. r0->r31, SREG, program counter (4 bytes), 2 bytes for
    * function parameters, 2 bytes for function arguments, and 2 bytes for
    * overflow checking
    *------------------------------------------------------------------------*/
   #define ARB_EXTRA_STACK_BYTES (45) /*To do: figure out why this needs to be
                                        45 instead of 43*/

   /*------------------------------------------------------------------------*
    * This 'mask' is used for detecting when a particular thread has blown
    * its allocated stack space.
    *------------------------------------------------------------------------*/
   #define ARB_STACK_OVERFLOW_CHECK (0xDEAD)

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "arb_error.h"
   #include "utl_linkedlist.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      ARB_STACK_64B   = 64,
      ARB_STACK_96B   = 96,
      ARB_STACK_128B  = 128,
      ARB_STACK_160B  = 160,
      ARB_STACK_192B  = 192,
      ARB_STACK_224B  = 224,
      ARB_STACK_256B  = 256,
      ARB_STACK_384B  = 384,
      ARB_STACK_512B  = 512,
      ARB_STACK_640B  = 640,
      ARB_STACK_768B  = 768,
      ARB_STACK_1024B = 1024,
      ARB_STACK_1152B = 1152,
      ARB_STACK_1280B = 1280,
      ARB_STACK_1408B = 1408,
      ARB_STACK_1536B = 1536,
      ARB_STACK_1792B = 1792,
      ARB_STACK_2048B = 2048,
      ARB_STACK_2304B = 2304,
      ARB_STACK_2560B = 2560,
      ARB_STACK_2816B = 2816,
      ARB_STACK_3072B = 3072,
      ARB_STACK_4096B = 4096

   }t_stackSize;

   typedef enum
   {
      READY = 0,
      RUNNING,
      BLOCKED,
      SLEEPING,
      INITIALIZED,
      TERMINATED

   }t_threadStatus;

   typedef uint8_t t_thrdPrio;   /*Thread priority 0->255 with 0 being highest*/

   typedef volatile int16_t t_THRDHANDLE;  /*Thread handle type*/

   typedef int16_t t_parameters; /*Thread function parameters*/

   typedef int16_t t_arguments;  /*Thread function arguments*/

   typedef struct TCB
   {
      /*--------------------------------------------------------------------*
       * Thread identification number.
       *--------------------------------------------------------------------*/
      uint8_t c_id;

      /*--------------------------------------------------------------------*
       * What state is this thread in?
       *--------------------------------------------------------------------*/
      t_threadStatus t_status;

      /*--------------------------------------------------------------------*
       * Priority level for this thread, 0 being the highest and 255 being
       * the lowest.
       *--------------------------------------------------------------------*/
      t_thrdPrio t_priority;

      /*--------------------------------------------------------------------*
       * After a call to sleep this value contains the system tick when the
       * thread will be re enabled.
       *--------------------------------------------------------------------*/
      uint16_t s_quantum;

      /*--------------------------------------------------------------------*
       * Starting address of the stack.
       *--------------------------------------------------------------------*/
      uint16_t s_stackStart;

      /*--------------------------------------------------------------------*
       * Ending address of the stack.
       *--------------------------------------------------------------------*/
      uint16_t s_stackEnd;

      /*--------------------------------------------------------------------*
       * Location of where the stack pointer last left off for this thread.
       * For the XMEGA, this location contains the program counter when a
       * thread is preempted by an interrupt or returns control over to the
       * OS. After performing a context save, this value will be increased
       * so that it spans all 32 registers and the SREG.
       *--------------------------------------------------------------------*/
      uint16_t s_sP;

      /*--------------------------------------------------------------------*
       * Storage location of the stack for this thread. After a context save,
       * the stack contains (in order) any values stored locally by the
       * thread, the program counter (PC), general purpose register 0 (r0),
       * SREG, and registers 1->31.
       *--------------------------------------------------------------------*/
      uint16_t *ps_stack;

   }t_tcb; /*The thread control block structure*/

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/
   extern t_CONTHNDL gt_activeThreads; /*Contains ptrs to the head, tail, and
                                         current thread running on the active
                                         list*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_THRDHANDLE arb_threadCreate( void *pt_function,
                                  t_parameters t_parms,
                                  t_arguments t_args,
                                  t_stackSize t_stack,
                                  t_thrdPrio t_priority);

   t_error arb_threadDestroy( t_THRDHANDLE t_thrdHandle);

   void arb_sleep( const uint16_t s_quantum ) __attribute__ ((noinline));

   void arb_updateSleepingThreads( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef arb_thread_h*/
