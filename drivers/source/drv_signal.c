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
 * File Name   : drv_signal.c
 *
 * Project     : Arbitros
 *               <https://code.google.com/p/arbitros/>
 *
 * Description : This file is responsible for controlling the signaling
 *               module which contains a red, yellow, and green LED.
 *
 * Last Update : Oct, 13, 2012
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Include Files
 *---------------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "arb_device.h"
#include "arb_semaphore.h"
#include "drv_signal.h"

/*---------------------------------------------------------------------------*
 * Private Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Datatypes
 *---------------------------------------------------------------------------*/
typedef struct
{
   /*------------------------------------------------------------------------*
    * Resources that can be shared amongst mutliple users (either a global
    * buffer or IO device) need to be protected against race conditions. We
    * use this semaphore for just that purpose.
    *------------------------------------------------------------------------*/
   t_SEMHANDLE t_mutex;

   /*------------------------------------------------------------------------*
    * The port containing the led's being controlled by this module.
    *------------------------------------------------------------------------*/
   uint8_t c_signalPort;

   /*------------------------------------------------------------------------*
    * The pin mask locations for the possible led's controlled by this driver.
    * Where index 0,1,2 refers to the red, yellow, and green led's
    * respectively.
    *------------------------------------------------------------------------*/
   uint8_t ac_ledPins[SIGNAL_MAX_LEDS];

   /*------------------------------------------------------------------------*
    * This mask turns on/off all the led's specified by 'ac_ledPins'.
    *------------------------------------------------------------------------*/
   uint8_t c_allLedMask;

   /*------------------------------------------------------------------------*
    * We are going to want to know how many 'handles' or users are attached to
    * this device.
    *------------------------------------------------------------------------*/
   uint8_t c_numUsers;

}t_signalDev;

/*---------------------------------------------------------------------------*
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static t_error signalOpen( t_DEVHANDLE t_devHandle);

static int32_t signalIoctl( t_DEVHANDLE t_devHandle,
                            uint16_t s_command,
                            int32_t  i_arguments);

static t_error signalClose( t_DEVHANDLE t_devHandle);

/*---------------------------------------------------------------------------*
 * Private Global Variables
 *---------------------------------------------------------------------------*/
t_deviceOperations gat_signalDevOps =
{
    signalOpen,
    NULL,
    NULL,
    signalIoctl,
    signalClose
};

/*---------------------------------------------------------------------------*
 * This is the device's shared memory, all actions on this variable must be
 * mutually exclusive.
 *---------------------------------------------------------------------------*/
static t_signalDev gt_signalDev;

/*---------------------------------------------------------------------------*
 * Inline Function Definitions
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * Private Function Definitions
 *---------------------------------------------------------------------------*/
t_error signalOpen( t_DEVHANDLE t_devHandle)
{

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_signalDev.t_mutex,
             0);

   gt_signalDev.c_numUsers++;

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_signalDev.t_mutex);

   return ARB_PASSED;

}/*End signalOpen*/

int32_t signalIoctl( t_DEVHANDLE t_devHandle,
                     uint16_t s_command,
                     int32_t  i_arguments)
{
   int32_t i_return = (int32_t)ARB_PASSED;

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_signalDev.t_mutex,
             0);

   switch( (t_signalCmd)s_command)
   {
      case RED_LED_ON:

         hal_gpioOff( gt_signalDev.c_signalPort,
                      gt_signalDev.ac_ledPins[0]);

      break;/*End case RED_LED_ON:*/

      case RED_LED_OFF:

         hal_gpioOn( gt_signalDev.c_signalPort,
                     gt_signalDev.ac_ledPins[0]);

      break;/*End case RED_LED_OFF:*/

      case RED_LED_TOGGLE:

         hal_gpioToggle( gt_signalDev.c_signalPort,
                         gt_signalDev.ac_ledPins[0]);

      break;/*End case RED_LED_TOGGLE:*/

      case GREEN_LED_ON:

         hal_gpioOff( gt_signalDev.c_signalPort,
                      gt_signalDev.ac_ledPins[2]);

      break;/*End case GREEN_LED_ON:*/

      case GREEN_LED_OFF:

         hal_gpioOn( gt_signalDev.c_signalPort,
                     gt_signalDev.ac_ledPins[2]);

      break;/*End case GREEN_LED_OFF:*/

      case GREEN_LED_TOGGLE:

         hal_gpioToggle( gt_signalDev.c_signalPort,
                         gt_signalDev.ac_ledPins[2]);

      break;/*End case GREEN_LED_TOGGLE:*/

      case YELLOW_LED_ON:

         hal_gpioOff( gt_signalDev.c_signalPort,
                      gt_signalDev.ac_ledPins[1]);

      break;/*End case YELLOW_LED_ON:*/

      case YELLOW_LED_OFF:

         hal_gpioOn( gt_signalDev.c_signalPort,
                     gt_signalDev.ac_ledPins[1]);

      break;/*End case YELLOW_LED_OFF:*/

      case YELLOW_LED_TOGGLE:

         hal_gpioToggle( gt_signalDev.c_signalPort,
                         gt_signalDev.ac_ledPins[1]);

      break;/*End case YELLOW_LED_TOGGLE:*/

      case ALL_LEDS_ON:

         hal_gpioOff( gt_signalDev.c_signalPort,
                      gt_signalDev.c_allLedMask);

      break;/*End case ALL_LEDS_ON:*/

      case ALL_LEDS_OFF:

         hal_gpioOn( gt_signalDev.c_signalPort,
                     gt_signalDev.c_allLedMask);

      break;/*End case ALL_LEDS_OFF:*/

      case ALL_LEDS_TOGGLE:

         hal_gpioToggle( gt_signalDev.c_signalPort,
                         gt_signalDev.c_allLedMask);

      break;/*End case ALL_LEDS_TOGGLE:*/

      default:

         i_return = (int32_t)ARB_INVALID_CMD;

      break;

   }/*End switch( (t_signalCmd)s_command)*/

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_signalDev.t_mutex);

   return i_return;

}/*End signalIoctl*/

/*---------------------------------------------------------------------------*
 * Remove this particular file attached to this device
 *---------------------------------------------------------------------------*/
t_error signalClose( t_DEVHANDLE t_devHandle)
{

   /*------------------------------------------------------------------------*
    * We are going to access global memory or a register, so perform lock
    *------------------------------------------------------------------------*/
   arb_wait( gt_signalDev.t_mutex,
             0);

   gt_signalDev.c_numUsers--;

   /*------------------------------------------------------------------------*
    * Release the lock
    *------------------------------------------------------------------------*/
   arb_signal( gt_signalDev.t_mutex);

   return ARB_PASSED;

}/*End signalClose*/

t_error drv_signalInit( t_signalSetup t_setup)
{
   t_error t_err = ARB_PASSED;
   t_gpioConf t_conf;
   int8_t c_index;

   /*------------------------------------------------------------------------*
    * Make sure the kernel is aware that a new device has been loaded.
    *------------------------------------------------------------------------*/
   t_err = arb_registerDevice( "signalDevice0",
                               arb_createDevId( t_setup.c_majorNum,
                               0),
                               &gat_signalDevOps);

   if( t_err < 0)
   {
      goto failed1;
   }

   /*------------------------------------------------------------------------*
    * Request a semaphore from the kernel. Since the signal port is a shared
    * resource we need to have all actions on it be mutually exclusive.
    *------------------------------------------------------------------------*/
   gt_signalDev.t_mutex = arb_semaphoreCreate( MUTEX);

   if( gt_signalDev.t_mutex < 0)
   {
      t_err = (t_error)gt_signalDev.t_mutex;
      goto failed2;

   }/*End if( gt_signalDev.t_mutex < 0)*/

   /*------------------------------------------------------------------------*
    * We dont have any users attached to this device
    *------------------------------------------------------------------------*/
   gt_signalDev.c_numUsers = 0;

   /*------------------------------------------------------------------------*
    * Configure the pins that control the led's...
    *------------------------------------------------------------------------*/
   gt_signalDev.c_signalPort = t_setup.c_signalPort;
   gt_signalDev.c_allLedMask = 0;

   for( c_index = 0; c_index < SIGNAL_MAX_LEDS; c_index++)
   {
      gt_signalDev.ac_ledPins[c_index] =
      t_setup.ac_ledPins[c_index];
      gt_signalDev.c_allLedMask |= t_setup.ac_ledPins[c_index];
   }

   t_conf.c_inputMask    = 0;
   t_conf.c_outputMask   = gt_signalDev.c_allLedMask;
   t_conf.b_setOutputLow = false;
   t_conf.t_outConf      = TOTEM;

   if( hal_configureGpioPort( t_setup.c_signalPort, t_conf) < 0)
   {
      t_err = ARB_HAL_ERROR;
      goto failed3;
   }

   /*------------------------------------------------------------------------*
    * Make sure the LED's are off - logic high is off...
    *------------------------------------------------------------------------*/
   hal_gpioOn( gt_signalDev.c_signalPort,
               gt_signalDev.c_allLedMask);

   return ARB_PASSED;

failed3:

   arb_semaphoreDestroy( gt_signalDev.t_mutex);

failed2:

   arb_destroyDevice( "signalDevice0");

failed1:

   return t_err;

}/*End drv_signalInit*/

/*---------------------------------------------------------------------------*
 * Remove the device from the system
 *---------------------------------------------------------------------------*/
void drv_signalExit( void)
{

   if( gt_signalDev.t_mutex != 0) /*If created... destroy*/
   {
      arb_semaphoreDestroy( gt_signalDev.t_mutex);
      arb_destroyDevice( "signalDevice0");

      /*------------------------------------------------------------------*
       * Remove any user-space specific generated memory here...
       *------------------------------------------------------------------*/
   }/*End if( gat_templateDev[c_index].t_mutex != 0)*/

}/*End drv_signalExit*/
