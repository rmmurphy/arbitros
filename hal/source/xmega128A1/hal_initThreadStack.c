/*---------------------------------------------------------------------------*
 * Copyright (C) 2014 Ryan M. Murphy (ryan.m.murphy.77@gmail.com)
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
 * File Name   : hal_initThreadStack.h
 *
 * Project     : Arbitros
 *               https://code.google.com/p/arbitros/
 *
 * Description : This file is responsible for initializing the thread stack
 *
 * Last Update : April 7, 2014
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include "arb_thread.h"

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
void hal_initThreadStack( void *pt_function,
                          t_parameters t_parms,
                          t_arguments t_args,
                          uint8_t *pc_stackPtr,
                          uint16_t s_endOfStackAdd,
                          uint16_t *ps_sP)
{
   uint32_t i_funAdd = 0;

   /*------------------------------------------------------------------------*
    * Make sure the stack pointer is pointing to the address of the thread's
    * call back function..for some reason we need to shift this address left
    * by 8
    *------------------------------------------------------------------------*/
   i_funAdd = (uint32_t)((uint16_t)pt_function);
   i_funAdd = i_funAdd << 8;

   *pc_stackPtr = i_funAdd & 0x000000FF;
   pc_stackPtr--;
   *pc_stackPtr = (i_funAdd >> 8) & 0x000000FF;
   pc_stackPtr--;
   *pc_stackPtr = (i_funAdd >> 16) & 0x000000FF;
   pc_stackPtr--;
   *pc_stackPtr = (i_funAdd >> 24) & 0x000000FF;
   pc_stackPtr--;

   /*------------------------------------------------------------------------*
    * Initialize the rest of the data on the stack so that the context
    * restore function works properly when we call this thread for the first
    * time.
    *------------------------------------------------------------------------*/
   *pc_stackPtr = 0x00; /*r0*/
   pc_stackPtr--;
   *pc_stackPtr = 0x80; /*SREG with interrupts enabled*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r1*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r2*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r3*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r4*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r5*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r6*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r7*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r8*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r9*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r10*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r11*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r12*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r13*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r14*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r15*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r16*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r17*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r18*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r19*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r20*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r21*/
   pc_stackPtr--;
   /*------------------------------------------------------------------------*
    * Function parameters are stored from left to right starting with CPU
    * register 25 and ending at CPU register 8.
    *------------------------------------------------------------------------*/
   *pc_stackPtr = (uint8_t)t_args;         /*r22*/
   pc_stackPtr--;
   *pc_stackPtr = (uint8_t)(t_args >> 8);  /*r23*/
   pc_stackPtr--;
   *pc_stackPtr = (uint8_t)t_parms;        /*r24*/
   pc_stackPtr--;
   *pc_stackPtr = (uint8_t)(t_parms >> 8); /*r25*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r26*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r27*/
   pc_stackPtr--;
   /*------------------------------------------------------------------------*
    * Since the stack frame pointer (Y registers) is another way of reading
    * and writing to the stack by the compiler, we need to make sure it is
    * assigned to the starting location (end of stack) of the stack.
    *------------------------------------------------------------------------*/
   *pc_stackPtr = (uint8_t)s_endOfStackAdd;        /*r28*/
   pc_stackPtr--;
   *pc_stackPtr = (uint8_t)(s_endOfStackAdd >> 8); /*r29*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r30*/
   pc_stackPtr--;
   *pc_stackPtr = 0x00; /*r31*/
   pc_stackPtr--;

   /*------------------------------------------------------------------------*
    * Save the stack pointer
    *------------------------------------------------------------------------*/
   *ps_sP = (uint16_t)pc_stackPtr;

}/*End arb_initThreadStack*/