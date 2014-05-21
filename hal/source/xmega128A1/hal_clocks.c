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
 * File Name   : hal_clocks.c
 *
 * Project     : Arbitros 
 *               https://code.google.com/p/arbitros/
 *               
 * Description : This file is responsible for setting up the processor clocks.
 *
 * Last Update : Nov 12, 2012
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include "hal_clocks.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef struct
{
   uint32_t i_cpuClock;

}t_clocks;

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
t_clocks gt_clocks;

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Public Function Definitions
 *---------------------------------------------------------------------------*/
uint32_t hal_getCpuFreq( void)
{
   return gt_clocks.i_cpuClock;
}

t_clocksError hal_setCpuFreq( uint32_t i_clockRate)
{
   /*------------------------------------------------------------------------*  
    * Enable internal 32 MHz ring oscillator and wait until it's stable.
    *------------------------------------------------------------------------*/   
   OSC.CTRL   = OSC_RC32MEN_bm; 
   
   CCP        = CCP_IOREG_gc;
   CLK.PSCTRL = CLK_PSADIV_1_gc;
   CLK.PSCTRL |= CLK_PSBCDIV_1_1_gc;

   while ( (OSC.STATUS & OSC_RC32MRDY_bm ) == 0);

   CCP      = CCP_IOREG_gc;
   CLK.CTRL = CLK_SCLKSEL_RC32M_gc;
   OSC.CTRL &= ~OSC_RC2MEN_bm;

   gt_clocks.i_cpuClock = i_clockRate;

   return CLOCKS_PASSED;

}/*End hal_setCpuFreq*/
