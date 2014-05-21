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
#ifndef hal_initThreadStack

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "arb_thread.h"
   
   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   void hal_initThreadStack( void *pt_function,
                             t_parameters t_parms,
                             t_arguments t_args,
                             uint8_t *pc_stackPtr,
                             uint16_t s_endOfStackAdd,
                             uint16_t *ps_sP);
                             
   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

 #endif/*End #ifndef hal_initThreadStack*/
