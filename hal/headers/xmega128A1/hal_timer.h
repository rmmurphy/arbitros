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
 * File Name   : hal_timer.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file provides and abstraction layer for controlling
 *               the timer module.
 *
 * Last Update : Oct 12, 2011
 *---------------------------------------------------------------------------*/
#ifndef hal_timer_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define hal_timer_h

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      TIMER_INV_GPIO_CONFIG = -15,/*Invalid GPIO configuration.*/
      TIMER_COMP_INVALID    = -14,/*Invalid compare value-greater than period*/
      TIMER_NO_CONFIG       = -13,/*Timer config hasn't been called*/
      TIMER_INVALID_PERIOD  = -12,/*Can't achieve the desired period.*/
      TIMER_INVALID_DIR     = -11,/*Invalid direction*/
      TIMER_INVALID_WF_MODE = -10,/*Invalid waveform generation mode.*/
      TIMER_MODULE_OPEN     = -9, /*Requesting a timer that is already open*/
      TIMER_INT_NOT_OPEN    = -8, /*Trying to access and interrupt that isn't
                                   mapped.*/
      TIMER_INTERRUPT_OPEN  = -7, /*Requesting and interrupt thats already
                                   mapped.*/
      TIMER_INVALID_COMP    = -6, /*Invalid compare channel request*/
      TIMER_INVALID_HANDLE  = -5, /*Handle doesn't map to a timer*/
      TIMER_INVALID_MODULE  = -4, /*Invalid timer module*/
      TIMER_PIN_IS_INPUT    = -3, /*Trying to access a pin as an output that is
                                   configured as an input*/
      TIMER_PIN_IS_OUTPUT   = -2, /*Trying to access a pin as an input that is
                                   configured as an output.*/
      TIMER_OUT_OF_HEAP     = -1, /*No more memory.*/
      TIMER_PASSED          = 0   /*Configuration good.*/

   }t_timerError;

   typedef enum
   {
      TIMER_1 = 0,  /*Timer/Counter C0*/
      TIMER_2 = 5,  /*Timer/Counter C1*/
      TIMER_3 = 10, /*Timer/Counter D0*/
      TIMER_4 = 15, /*Timer/Counter D1*/
      TIMER_5 = 20, /*Timer/Counter E0*/
      TIMER_6 = 25, /*Timer/Counter E1*/
      TIMER_7 = 30, /*Timer/Counter F0*/
      TIMER_8 = 35  /*Timer/Counter F1*/

   }t_timerModId;

   typedef enum
   {

      COMPAREA = 0,
      COMPAREB,
      COMPAREC,
      COMPARED,
      OVERFLOW

   }t_compType;

   typedef enum
   {

      NORMAL                = 0,
      FREQ_GEN,
      UNDEFINED,
      SINGLE_SLOPE,
      DUAL_SLOPE_TOP,
      DUAL_SLOPE_TOP_BOTTOM,
      DUAL_SLOPE_BOTTOM

   }t_wfMode;

   typedef enum
   {
      DIRECTION_UP   = 0,
      DIRECTION_DOWN,
   }t_countDir;

   typedef struct
   {
      /*---------------------------------------------------------------------*
       * The type of timer being configured.
       *---------------------------------------------------------------------*/
      t_wfMode t_mode;

      /*---------------------------------------------------------------------*
       * The direction the timer counts.
       *---------------------------------------------------------------------*/
      t_countDir t_dir;

      /*---------------------------------------------------------------------*
       * The period of the particular timer being configured.
       *---------------------------------------------------------------------*/
      float f_period;

   }t_timerConfig;

   typedef volatile int16_t t_TIMERHNDL; /*Handle to a particular timer*/

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_timerError hal_requestTimerInterrupt( t_TIMERHNDL t_handle,
                                           t_compType  t_type,
                                           void (*pf_funPtr)( void));

   t_timerError hal_releaseTimerInterrupt( t_TIMERHNDL t_handle,
                                           t_compType  t_type);

   t_timerError hal_configureTimer( t_TIMERHNDL t_handle,
                                    t_timerConfig t_conf);

   t_TIMERHNDL hal_requestTimer( t_timerModId t_id);

   t_timerError hal_releaseTimer( t_TIMERHNDL t_handle);

   t_timerError hal_startTimer( t_TIMERHNDL t_handle);

   t_timerError hal_stopTimer( t_TIMERHNDL t_handle);

   int32_t hal_getTimerCount( t_TIMERHNDL t_handle);

   t_timerError hal_setCompareValue( t_TIMERHNDL t_handle,
                                     t_compType  t_type,
                                     uint16_t    s_value);

   int32_t hal_getCompareValue( t_TIMERHNDL t_handle,
                                t_compType  t_type);

   int16_t hal_getIntStatus( t_TIMERHNDL t_handle,
                             t_compType  t_type);

   t_timerError hal_clearIntStatus( t_TIMERHNDL t_handle,
                                    t_compType  t_type);

   t_timerError hal_enableCompareChannel( t_TIMERHNDL t_handle,
                                          t_compType t_type,
                                          bool b_ouputOnPin);

   t_timerError hal_disableCompareChannel( t_TIMERHNDL t_handle,
                                           t_compType  t_type);

   int32_t hal_getPeriodValue( t_TIMERHNDL t_handle);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef hal_timer_h*/
