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
 * File Name   : arb_error.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file defines the set of possible OS errors.
 *
 * Last Update : June, 12, 2011
 *---------------------------------------------------------------------------*/
#ifndef arb_error_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define arb_error_h

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      ARB_OPEN_ERROR       = -16,/*A file failed to open*/
      ARB_MBX_EMPTY        = -15,/*A mailbox queue is empty*/
      ARB_MBX_FULL         = -14,/*A mailbox queue is full*/
      ARB_SEM_DEC_ERROR    = -13,/*Semaphore failed to decrement*/
      ARB_READ_ERROR       = -12,/*Read command failed*/
      ARB_WRITE_ERROR      = -11,/*Write command failed*/
      ARB_HAL_ERROR        = -10,/*A HAL component failed initialization*/
      ARB_DEVICE_NOT_FOUND = -9, /*The device driver is not found*/
      ARB_INVALID_PRIORITY = -8, /*Threads can't share the same priority*/
      ARB_INVALID_HANDLE   = -7, /*Handle not found*/
      ARB_INVALID_ARG      = -6, /*Argument out of range*/
      ARB_INVALID_CMD      = -5, /*Invalid command*/
      ARB_NAME_ERROR       = -4, /*Device name too long*/
      ARB_NULL_PTR         = -3, /*Referenced a NULL pointer*/
      ARB_DEVICE_PRESENT   = -2, /*Trying to create a device that is already
                                   present*/
      ARB_OUT_OF_HEAP      = -1, /*Ran out of memory*/
      ARB_PASSED           =  0  /*No error*/

   }t_error;

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   void arb_stackOverflow( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef arb_error_h*/
