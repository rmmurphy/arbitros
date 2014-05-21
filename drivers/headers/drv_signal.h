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
 * File Name   : drv_signal.h
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for controlling the signaling
 *               module which contains a red, yellow, and green LED.
 *
 * Last Update : Oct, 13, 2012
 *---------------------------------------------------------------------------*/
#ifndef drv_signal_h

   #ifdef __cplusplus
   extern "C" {
   #endif

   /*------------------------------------------------------------------------*
    * Global Defines
    *------------------------------------------------------------------------*/
   #define drv_signal_h
   #define SIGNAL_MAX_LEDS (3) /*The max number of Led's controlled*/

   /*------------------------------------------------------------------------*
    * Include Files
    *------------------------------------------------------------------------*/
   #include "avr_compiler.h"
   #include "hal_gpio.h"

   /*------------------------------------------------------------------------*
    * Global Typedefs
    *------------------------------------------------------------------------*/
   typedef enum
   {
      RED_LED_ON = 0,
      RED_LED_OFF,
      RED_LED_TOGGLE,
      GREEN_LED_ON,
      GREEN_LED_OFF,
      GREEN_LED_TOGGLE,
      YELLOW_LED_ON,
      YELLOW_LED_OFF,
      YELLOW_LED_TOGGLE,
      ALL_LEDS_ON,
      ALL_LEDS_OFF,
      ALL_LEDS_TOGGLE

   }t_signalCmd;

   typedef struct
   {

      /*---------------------------------------------------------------------*
       * The GPIO port containing the pins that control the led's.
       *---------------------------------------------------------------------*/
       uint8_t c_signalPort;

      /*---------------------------------------------------------------------*
       * The pin mask locations for the possible led's controlled by this
       * driver. Where index 0,1,2 refers to the red, yellow, and green led's
       * respectively.
       *---------------------------------------------------------------------*/
       uint8_t ac_ledPins[SIGNAL_MAX_LEDS];

      /*---------------------------------------------------------------------*
       * This value represents a unique number assigned to this driver and has
       * to be different from all others registered with the kernel.
       *---------------------------------------------------------------------*/
      uint8_t c_majorNum;

   }t_signalSetup;

   /*------------------------------------------------------------------------*
    * Global Variables
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Inline functions
    *------------------------------------------------------------------------*/

   /*------------------------------------------------------------------------*
    * Global Function Prototypes
    *------------------------------------------------------------------------*/
   t_error drv_signalInit( t_signalSetup t_setup);

   void drv_signalExit( void);

   #ifdef __cplusplus
   }/*End extern "C"*/
   #endif

#endif/*End #ifndef drv_signal_h*/
