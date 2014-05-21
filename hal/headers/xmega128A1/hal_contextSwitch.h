/*---------------------------------------------------------------------------*
 * Copyright (C) 2014 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
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
 * File Name   : hal_contextSave.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for context switching during
 *
 * References  : 1) 'Multitasking on an AVR' Example C Implementation of a
 *                  Multitasking Kernel for the AVR, by Richard Barry
 *               2) http://www.freertos.org/
 *
 * Last Update : April, 4, 2014
 *---------------------------------------------------------------------------*/
#ifndef hal_contextSave_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define hal_contextSave_h

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "arb_thread.h"
   #include "arb_error.h"
   
   t_tcb *pt_csTemp;
   
   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/
   static inline void __attribute__((always_inline)) hal_switchToKernelStack\
   ( void)
   {
      /*---------------------------------------------------------------------*
       * The kernel stack is located at the end of SRAM
       *---------------------------------------------------------------------*/
      SPL = (uint8_t)(RAMEND & 0x00FF);
      SPH = (uint8_t)((RAMEND >> 8) & 0x00FF);

      /*---------------------------------------------------------------------*
       * Point the frame pointer (Y register) to the new stack location
       *---------------------------------------------------------------------*/
      asm("lds r28, 0x3D");
      asm("lds r29, 0x3E");

   }/*End hal_switchToKernelStack*/

   /*------------------------------------------------------------------------*
    * This function assumes that it has been called within the context of
    * mutual exclusion. Therefore, it makes sure that the SREG is saved with
    * the GIE bit set. Interrupts will not get turned back on until a thread
    * with the GIE bit enabled is restored with the call to
    * 'hal_contextRestore'.
    *------------------------------------------------------------------------*/
   static inline void __attribute__((always_inline)) hal_contextSaveWithIntsOn\
   ( void)
   {

      /*---------------------------------------------------------------------*
       * !Note! The XMEGA pushes the program counter onto the stack when a
       * thread gets preempted or makes a subroutine call. Upon entering this
       * this routine the stack pointer is pointing to the thread's
       * program counter.
       *---------------------------------------------------------------------*/

      /*---------------------------------------------------------------------*
       * Push the rest of the XMEGA registers SREG, and r0->r31 onto the
       * thread's stack (starting with the first address after the PC).
       * After storing R1 we need to set it to zero so we are performing the
       * same operation as the compiler if it were explicitly doing the
       * context save for us.
       *---------------------------------------------------------------------*/
      asm(" push r0");

      /*---------------------------------------------------------------------*
       * We disabled interrupts before calling this function, therefore we
       * need to make sure that the SREG is saved with the GIE bit set.
       *---------------------------------------------------------------------*/
      asm( "in r0, __SREG__");
      asm(" push r16");        /*Save the contents of r16 so we can use it*/
      asm( "mov r16, r0");     /*Can't directly set a bit in r0 so copy it to
                                r16*/
      asm( "sbr r16, 128");    /*Turn interrupts back on in the stored SREG*/
      asm( "mov r0, r16");     /*Copy the change back into r0*/
      asm( "pop r16");         /*Get back the old value of r16*/
      asm( "push r0");         /*Save SREG with GIE enabled*/
      asm( "push r1");
      asm( "clr r1");
      asm( "push r2");
      asm( "push r3");
      asm( "push r4");
      asm( "push r5");
      asm( "push r6");
      asm( "push r7");
      asm( "push r8");
      asm( "push r9");
      asm( "push r10");
      asm( "push r11");
      asm( "push r12");
      asm( "push r13");
      asm( "push r14");
      asm( "push r15");
      asm( "push r16");
      asm( "push r17");
      asm( "push r18");
      asm( "push r19");
      asm( "push r20");
      asm( "push r21");
      asm( "push r22");
      asm( "push r23");
      asm( "push r24");
      asm( "push r25");
      asm( "push r26");
      asm( "push r27");
      asm( "push r28");
      asm( "push r29");
      asm( "push r30");
      asm( "push r31");

      /*---------------------------------------------------------------------*
       * Now save the stack pointer low byte first, then high byte for the
       * currently running thread. The stack pointer will be pointing to r31
       *---------------------------------------------------------------------*/
      pt_csTemp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR_CONT_CURR(gt_activeThreads);
      pt_csTemp->s_sP = (0x00FF & (uint16_t)SPL);
      pt_csTemp->s_sP |= (((uint16_t)SPH << 8) & 0xFF00);

   }/*End hal_contextSaveWithIntsOn*/

   /*------------------------------------------------------------------------*
    * This function assumes that it has been called with interrupts purposely
    * disabled. Therefore, it makes sure that the SREG is saved with the GIE
    * bit set. Interrupts will not get turned back on until a thread with the
    * GIE bit enabled is restored with the call to 'hal_contextRestore'.
    *------------------------------------------------------------------------*/
   static inline void __attribute__((always_inline)) \
   hal_contextSaveWithIntsOff( void)
   {

      /*---------------------------------------------------------------------*
       * !Note! The XMEGA pushes the program counter onto the stack when a
       * thread gets preempted or makes a subroutine call. Upon entering this
       * this routine the stack pointer is pointing to the thread's
       * program counter.
       *---------------------------------------------------------------------*/

      /*---------------------------------------------------------------------*
       * Push the rest of the XMEGA registers SREG, and r0->r31 onto the
       * thread's stack (starting with the first address after the PC).
       * After storing R1 we need to set it to zero so we are performing the
       * same operation as the compiler if it were explicitly doing the
       * context save for us.
       *---------------------------------------------------------------------*/
      asm(" push r0");

      /*---------------------------------------------------------------------*
       * We disabled interrupts before calling this function, therefore we
       * need to make sure that the SREG is saved with the GIE bit set.
       *---------------------------------------------------------------------*/
      asm( "in r0, __SREG__");
      asm( "push r0");         /*Save SREG*/
      asm( "push r1");
      asm( "clr r1");
      asm( "push r2");
      asm( "push r3");
      asm( "push r4");
      asm( "push r5");
      asm( "push r6");
      asm( "push r7");
      asm( "push r8");
      asm( "push r9");
      asm( "push r10");
      asm( "push r11");
      asm( "push r12");
      asm( "push r13");
      asm( "push r14");
      asm( "push r15");
      asm( "push r16");
      asm( "push r17");
      asm( "push r18");
      asm( "push r19");
      asm( "push r20");
      asm( "push r21");
      asm( "push r22");
      asm( "push r23");
      asm( "push r24");
      asm( "push r25");
      asm( "push r26");
      asm( "push r27");
      asm( "push r28");
      asm( "push r29");
      asm( "push r30");
      asm( "push r31");

      /*---------------------------------------------------------------------*
       * Now save the stack pointer low byte first, then high byte for the
       * currently running thread. The stack pointer will be pointing to r31
       *---------------------------------------------------------------------*/
      pt_csTemp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR_CONT_CURR(gt_activeThreads);
      pt_csTemp->s_sP = (0x00FF & (uint16_t)SPL);
      pt_csTemp->s_sP |= (((uint16_t)SPH << 8) & 0xFF00);

   }/*End hal_contextSaveWithIntsOff*/

   static inline void __attribute__((always_inline)) hal_contextRestore( void)
   {

      /*---------------------------------------------------------------------*
       * Get the current thread's stack pointer from the TCB and load the
       * CPU stack register
       *---------------------------------------------------------------------*/
      pt_csTemp = (t_tcb *)UTL_GET_LINK_ELEMENT_PTR_CONT_CURR(gt_activeThreads);
      SPL = (uint8_t)((pt_csTemp->s_sP) & 0x00FF);
      SPH = (uint8_t)(((pt_csTemp->s_sP) >> 8) & 0x00FF);

      /*---------------------------------------------------------------------*
       * Now that the CPU stack ptr is pointing to the stack for this thread,
       * restore the general purpose registers to the place where the thread
       * last left off by popping there values off the thread's stack.
       *---------------------------------------------------------------------*/
      asm( "pop r31");
      asm( "pop r30");
      asm( "pop r29");
      asm( "pop r28");
      asm( "pop r27");
      asm( "pop r26");
      asm( "pop r25");
      asm( "pop r24");
      asm( "pop r23");
      asm( "pop r22");
      asm( "pop r21");
      asm( "pop r20");
      asm( "pop r19");
      asm( "pop r18");
      asm( "pop r17");
      asm( "pop r16");
      asm( "pop r15");
      asm( "pop r14");
      asm( "pop r13");
      asm( "pop r12");
      asm( "pop r11");
      asm( "pop r10");
      asm( "pop r9");
      asm( "pop r8");
      asm( "pop r7");
      asm( "pop r6");
      asm( "pop r5");
      asm( "pop r4");
      asm( "pop r3");
      asm( "pop r2");
      asm( "pop r1");

      /*---------------------------------------------------------------------*
       * The pop of this stack location into r0 contains the SREG
       *---------------------------------------------------------------------*/
      asm( "pop r0");

      /*---------------------------------------------------------------------*
       * Now restore the thread's old SREG value using the 'out' cmd. This cmd
       * is used whenever we write to an IO location.
       *---------------------------------------------------------------------*/
      asm( "out __SREG__, r0");

      /*---------------------------------------------------------------------*
       * This pop contains the thread's saved r0 value
       *---------------------------------------------------------------------*/
      asm( "pop r0");

   }/*End hal_contextRestore*/
   
   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef hal_contextSave_h*/