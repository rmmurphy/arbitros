/*---------------------------------------------------------------------------*
 * Copyright (C) 2011-2012 Ryan M. Murphy <ryan.m.murphy.77@gmail.com>
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * File Name   : usr_console.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for command and control over the
 *               console module. This involves creating the threads,
 *               and semaphores, which provide direct access to the console 
 *               from a user space application.
 *
 * Programmer  : Ryan M Murphy
 *
 * Last Update : Dec, 10, 2011
 *---------------------------------------------------------------------------*/
#ifndef usr_console_h

   #ifdef __cplusplus
   extern "C" {
   #endif
   
   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define usr_console_h
   
   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "arb_device.h"
   #include "drv_console.h"

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   bool usr_console( t_DEVHANDLE t_consoleHndl,
                     int8_t *pc_buff,
                     t_consoleTokHndl *pt_tokHndl);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif
   
#endif/*End #ifndef usr_console_h*/
