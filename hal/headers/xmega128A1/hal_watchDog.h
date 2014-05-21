/*---------------------------------------------------------------------------*
 * Copyright (C) 2012 Ryan M. Murphy (ryan.m.murphy.77@gmail.com)
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
 * File Name   : hal_watchDog.h
 *
 * Project     : Arbitros
 *               https://code.google.com/p/arbitros/
 *
 * Description : This file is responsible for controlling access to the
 *               watchdog timer.
 *
 * Last Update : Nov 21, 2012
 *---------------------------------------------------------------------------*/
#ifndef hal_watchDog_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define HAL_WD_IS_SYNC_BUSY() ( WDT.STATUS & WDT_SYNCBUSY_bm )
   #define HAL_WD_RESET() asm("wdr")

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      WD_NO_CONFIG      = -5, /*hal_configureWd has not been called.*/
      WD_INVALID_PERIOD = -4, /*Period out of range.*/
      WD_UNAVAILABLE    = -3, /*Handle already opened for the WD controller*/
      WD_INVALID_HANDLE = -2, /*Handle doesn't map to a WD controller*/
      WD_OUT_OF_HEAP    = -1, /*No more memory.*/
      WD_PASSED         = 0   /*Configuration good.*/

   }t_wdError;

   typedef struct
   {
      /*---------------------------------------------------------------------*
       * The period of the watchdog timer in milliseconds.
       *---------------------------------------------------------------------*/
      uint32_t i_period;

   }t_wdConfig; /*Configuration for a the watchdog controller*/

   typedef volatile int16_t t_WDHNDL; /*Handle to a particular WD*/

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * This function request access to the watchdog timer.
    *------------------------------------------------------------------------*/
   t_WDHNDL hal_requestWdAccess( void);

   /*------------------------------------------------------------------------*
    * This function releases access over the watchdog.
    *------------------------------------------------------------------------*/
   t_wdError hal_releaseWdAccess( t_WDHNDL t_handle);

   /*------------------------------------------------------------------------*
    * Configures the DMA transaction for the channel pointed to by
    * 't_handle'.
    *------------------------------------------------------------------------*/
   t_wdError hal_configureWd( t_WDHNDL t_handle,
                              t_wdConfig t_conf);

   /*------------------------------------------------------------------------*
    * This function enables the watchdog timer.
    *------------------------------------------------------------------------*/
    t_wdError hal_wdEnable( t_WDHNDL t_handle);

   /*------------------------------------------------------------------------*
    * This function disables the watchdog timer.
    *------------------------------------------------------------------------*/
    t_wdError hal_wdDisable( t_WDHNDL t_handle );

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

 #endif/*End #ifndef hal_watchDog_h*/
