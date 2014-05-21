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
 * File Name   : hal_pmic.h
 *
 * Project     : Arbitros
 *               https://code.google.com/p/arbitros/
 *
 * Description : This file is responsible for abstracting control over
 *               a device specific programmable interrupt controller.
 *
 * Last Update : September 26, 2012
 *---------------------------------------------------------------------------*/
#ifndef hal_pmic_h

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
   #define hal_pmic_h
   #define HAL_BEGIN_CRITICAL( ) AVR_ENTER_CRITICAL_REGION();
   #define HAL_END_CRITICAL( ) AVR_LEAVE_CRITICAL_REGION();
   #define HAL_SEI( ) __asm__ __volatile__ ("sei" ::);
   #define HAL_CLI( ) __asm__ __volatile__ ("cli" ::);
   #define HAL_RETI( ) __asm__ __volatile__ ("reti" ::);
   #define HAL_RET( ) __asm__ __volatile__ ("ret" ::);
   #define HAL_ARE_INTS_EN( ) (CPU_SREG & 0x80)
   #define HAL_IS_ACTIVE_INT( ) (PMIC_STATUS)

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      INT_LEVEL_0 = 1,
      INT_LEVEL_1 = 2,
      INT_LEVEL_2 = 4
   }t_pmicLevel;

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   void hal_configureIntLevel(t_pmicLevel t_level);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef hal_pmic_h*/
