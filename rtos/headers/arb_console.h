/*---------------------------------------------------------------------------*
 * Copyright (C) 2011-2013 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
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
 * File Name   : arb_console.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for command and control over the
 *               console module from a RTOS perspective. This involves
 *               creating the threads and semaphores which provide direct
 *               access to the console driver for both a kernel space and
 *               user space application.
 *
 * Programmer  : Ryan M Murphy
 *
 * Last Update : Jan, 14, 2013
 *---------------------------------------------------------------------------*/
#ifndef arb_console_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define arb_console_h
   #define MAX_CONSOLE_BUFF_SIZE (300)

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "drv_console.h"
   #include "arb_device.h"
   #include "arb_thread.h"

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_error arb_consoleInit( char *pc_consDriver,
                            char *pc_sdDriver,
                            t_stackSize t_stack,
                            t_thrdPrio t_pri,
                            bool (*pf_funPtr)( t_DEVHANDLE t_consoleHndl,
                                               int8_t *pc_buff,
                                               t_consoleTokHndl *pt_tokHndl));

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef arb_console_h*/
