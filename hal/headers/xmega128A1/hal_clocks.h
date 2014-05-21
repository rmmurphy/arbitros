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
 * File Name   : hal_clocks.h
 *
 * Project     : Arbitros
 *               https://code.google.com/p/arbitros/
 *
 * Description : This file is responsible for setting up the processor clocks.
 *
 * Last Update : Nov 12, 2012
 *---------------------------------------------------------------------------*/
#ifndef hal_clocks_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define hal_clocks_h
 
   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "hal_pmic.h"

   /*------------------------------------------------------------------------*
    * Global typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      CLOCKS_INVALID_FREQ = -1, /*Can't set CPU freq.*/
      CLOCKS_PASSED       = 0   /*Configuration good.*/

   }t_clocksError;

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Inline functions
    *------------------------------------------------------------------------*/
   static inline void __attribute__((always_inline)) 
   hal_busyDelayMs(uint32_t i_msec, uint32_t i_cpuSpeed) 
   {

	   uint16_t __ticks;
	   double __tmp = ((i_cpuSpeed) / 4e3) * i_msec;
  
      HAL_BEGIN_CRITICAL();

	   if (__tmp < 1.0)
	   __ticks = 1;
	   else if (__tmp > 65535)
	   {
   	   //	__ticks = requested delay in 1/10 ms
   	   __ticks = (uint16_t) (i_msec * 10);
   	   while(__ticks)
   	   {
      	   // wait 1/10 ms
      	   _delay_loop_2(((F_CPU) / 4e3) / 10);
      	   __ticks --;
   	   }
   	   return;
	   }
	   else
	      __ticks = (uint16_t)__tmp;

	   __asm__ volatile (
	   "1: sbiw %0,1" "\n\t"
	   "brne 1b"
	   : "=w" (__ticks)
	   : "0" (__ticks)
	   );

      HAL_END_CRITICAL();

   }/*End hal_busyDelayMs*/

   static inline void __attribute__((always_inline))
   hal_busyDelayUs(uint32_t i_usec, uint32_t i_cpuSpeed)
   {
      uint8_t __ticks;
      double __tmp = ((i_cpuSpeed) / 3e6) * i_usec;
      
      HAL_BEGIN_CRITICAL();

      if (__tmp < 1.0)
         __ticks = 1;
      else if (__tmp > 255)
      {
         _delay_ms(i_usec / 1000.0);
         return;
      }
      else
         __ticks = (uint8_t)__tmp;
      _delay_loop_1(__ticks);

      HAL_END_CRITICAL();

   }/*End hal_busyDelayUs*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_clocksError hal_setCpuFreq( uint32_t i_cpuFreq);
   uint32_t hal_getCpuFreq( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef hal_clocks_h*/
